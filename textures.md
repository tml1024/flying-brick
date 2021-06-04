Texture workflow

The Blender file refers to texture images (just one in this silly
sample) somewhere, in this case in
PackageSources\SimObjects\Airplanes\FOO\texture.

Exporting to glTF puts identical copies in the same folder as where
the .gltf file is stored, in
PackageSources\SimObjects\Airplanes\FOO\model. (This can be
changed in the export dialog.)

The Project Editor's Build All converts PNG to DDS textures and puts
them in Packages\BAR-aircraft-MUMBLE\SimObjects\Airplanes\FOO\texture.

But! It isn't the copies that the exporter puts in the model folder
that the Build All uses. It looks for the files in the texture folder.
