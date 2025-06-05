Code utilisé pour les PAMIs

The Boot button need to be held down while connecting the ESP-S3-Zero to be able to upload.
Once connected in boot mode the used COM port while change you might need to change the chosen COM port  in VS code.
When in boot mode the program while not work you need to unplug and replug the ESP-S3-Zero to exit boot mode.

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
