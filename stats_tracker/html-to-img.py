import imgkit

with open('loc.svg') as f:
  imgkit.from_file(f, 'build/loc.png')
