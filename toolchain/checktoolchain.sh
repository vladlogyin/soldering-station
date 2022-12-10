 #!/bin/bash

# There are versions of GDB that support multitarget, but not all of them do, currently assume the OS shipped one doesn't
COMPILER="gcc g++ as ld objcopy objdump gdb"
TOOLS="openocd"

check_binaries()
{
 error=0
  for compiler in $COMPILER
    do
      COMPILERPATH=$(which $CROSS$compiler 2>/dev/null)
      if [ $? -eq 0 ]; then
        echo "$CROSS$compiler found: $COMPILERPATH"
      else
        echo "$CROSS$compiler not found"
        error=1
      fi
    done

    # Check if all other tool binaries can be found
    for tool in $TOOLS
    do
      TOOLPATH=$(which $tool 2>/dev/null)
      if [ $? -eq 0 ]; then
        echo "$tool found: $TOOLPATH"
      else
        echo "$tool not found"
        error=1
      fi
    done
    return $error
}

do_portage_install()
{
  # Check if sudo is installed
  which sudo
  if [ $? -eq 0 ]; then

    echo "Checking package USE flags"
    if not grep -Rxq "dev-embedded/openocd.*" /etc/portage/package.use
    then
      echo "openocd USE flags not found"
      printf "\n#Added automatically\ndev-embedded/openocd ftdi jlink parport" | sudo tee -a /etc/portage/package.use/cross-zz >/dev/null
      if [ $? -ne 0 ]; then
        return 11
      fi
    fi
    if not grep -Rxq "cross-arm-none-eabi/newlib.*nano.*" /etc/portage/package.use
    then
      echo "newlib USE flags not found"
      printf "\n#Added automatically\ncross-arm-none-eabi/newlib nano" |\
sudo tee -a /etc/portage/package.use/cross-zz >/dev/null
      if [ $? -ne 0 ]; then
        return 11
      fi
    fi

    which openocd
    if [ $? -eq 0 ]; then
      echo "openocd is already installed"
    else
      echo "Installling openocd"
      sudo emerge -a dev-embedded/openocd
    fi
    # Check if crossdev is installed
    which crossdev
    if [ $? -eq 0 ]; then
      echo "crossdev is already installed"
    else
      echo "Installling crossdev"
      sudo emerge -a sys-devel/crossdev
      which crossdev
      if [ $? -ne 0 ]; then
        echo "Failed to install crossdev"
        return 2
      fi
    fi
    echo "Building toolchain using crossdev"
    sudo crossdev -S -s4 --ex-gdb -t arm-none-eabi --genv 'EXTRA_ECONF="--with-multilib-list=rmprofile"'
    if [ $? -ne 0 ]; then
      echo "Failed to build toolchain"
      return 3
    fi
    return 0
  fi
  return 1
}

do_pacman_install()
{
  # Check if sudo is installed
  which sudo
  if [ $? -eq 0 ]; then
    # Trigger database update
    sudo pacman -Syy
    sudo pacman -S arm-none-eabi-gcc arm-none-eabi-binutils \
arm-none-eabi-gdb arm-none-eabi-newlib openocd
    return $?
  elif [ $(id -u) -eq 0 ]; then
    # We're root, so we can still proceed
    pacman -Syy
    pacman -S arm-none-eabi-gcc arm-none-eabi-binutils arm-none-eabi-gdb \
arm-none-eabi-newlib openocd
    return $?
  else
    # We can't really do anything except complain
    echo "You need to be root or have sudo set up for this script to \
automatically install the required packages"
    return 1
  fi
}

do_apt_install()
{
  sudo apt update
  sudo apt install gcc-arm-none-eabi openocd gdb-multiarch
}

do_wsl_extras()
{
  openocd_curl=$( \
curl -s https://api.github.com/repos/openocd-org/\
openocd/releases/latest | \
grep -o "openocd.*w64.*.tar.gz" | cut -d '"' -f 4 )
  openocd_download="/tmp/"
  openocd_download+=$( echo "$openocd_curl" | \
grep -ox "openocd-v[^[:space:]]*.tar.gz" )
  openocd_url=$( echo "$openocd_curl" | grep -ox "openocd-org.*.tar.gz" )
  wget "https://github.com/$openocd_url" -O "$openocd_download"
  script_path=$(dirname -- "${BASH_SOURCE[0]}")
  mkdir "$script_path/openocd-win"
  tar -xvf "$openocd_download" -C "$script_path/openocd-win/"
  rm "$openocd_download"
  # Add the root certificate of the driver
  certutil.exe -addstore -v root $script_path/stlink-win/ugr.crt
  # Install the driver
  pnputil.exe -i -a $script_path/stlink-win/stlink_dbg_winusb.inf
}

echo Checking toolchain for $1

case $1 in
  "WSL"|"Linux")
    if [ $1 == "WSL" ]; then
      do_wsl_extras
    fi
    # Check if all required compiler binaries can be found
    CROSS="arm-none-eabi-"
    check_binaries
    if [ $? -eq 0 ]; then
      echo "Toolchain binaries are present"
    else
      # Try to figure out which distro we're on in order to
      # install packages using the right manager
      which lsb_release
      if [ $? -eq 0 ]; then
        DISTRO=$(lsb_release -is)
        case $DISTRO in
          "LinuxMint"|"Mint"|"Ubuntu"|"Debian")
            # Debian & derivatives use aptitude as their package manager
            do_apt_install
          ;;
          "Gentoo")
            do_portage_install
          ;;
          "Artix"|"artix"|"artixlinux"|"Artix release"|"Artix Linux"|\
          "ManjaroLinux"|"Arch"|"archlinux"|"arch"|"Arch Linux")
           # Arch & derivatives use pacman
            do_pacman_install
          ;;
        esac
      fi

    fi
  ;;
  "Darwin")
    if [ $(uname -m) != "x86_64" ]; then
      echo "only AMD64 is supported"
      exit 1
    fi
    which brew
    # If brew is not found in $PATH, try installing it
    if [ $? -eq 0 ]; then
      echo "Homebrew is installed"
    else
      echo "Running brew install script"
      bash -c "$(curl -fsSL \
      https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    fi
    # Check if the installer script errored out
    if [ $? -eq 0 ]; then
      echo "Homebrew is installed"
    else
      echo "Error installing brew"
      exit 2
    fi
    # Assume brew is successfully installed
    brew tap osx-cross/arm
    brew install arm-gcc-bin open-ocd gdb
    # This should create a functioning install of gcc as arm-none-eabi-*
    # Check this is actually true
    CROSS="arm-none-eabi-"
    check_binaries
    if [ $? -eq 0 ]; then
      echo "Toolchain binaries are present"
    else
      echo "Something went wrong! Toolchain binaries are not present"
      exit 3
    fi
  ;;
  *)
    echo "$1 is unsupported"
  ;;
esac
