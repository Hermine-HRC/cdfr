Code utilisé pour les PAMIs

# Testing

## Requirements

> **_NOTE:_**
> These requirements are already respected if ROS2 is installed.

```bash
sudo apt install uncrustify # for Linux
# for Windows see: https://github.com/Glavin001/atom-beautify/issues/772#issuecomment-216685220

pip3 install ament-style-uncrustify pytest
```

## Usage

To be launched from the *pami* folder.

```bash
pytest test/test_uncrustify.py
```
