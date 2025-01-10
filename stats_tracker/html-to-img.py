import imgkit

with open('loc.svg') as f:
  options = {
    "enable-local-file-access": ""
  }
  imgkit.from_file(f, 'build/loc.png', options)
