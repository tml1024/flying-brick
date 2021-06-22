Texture workflow

The Blender file refers to texture images (just one in this silly
sample) somewhere, in this case in
PackageSources\SimObjects\Airplanes\FOO\texture.

Exporting to glTF puts identical copies in the same folder as where
the .gltf file is stored, in
PackageSources\SimObjects\Airplanes\FOO\model. (This can be
changed in the export dialog.)

The names of texture PNG files should not contain multiple periods.
(Noticed in another project where I tried to use
"Propeller.baked.png".) The glTF exporter converts extra periods to
underscores when it copies them to the model folder, and in the
references in the glTF file.

The Project Editor's Build All converts PNG to DDS textures and puts
them in Packages\BAR-aircraft-MUMBLE\SimObjects\Airplanes\FOO\texture.

But! It isn't the copies that the exporter puts in the model folder
that the Build All uses. It looks for the files in the texture folder.
This is why the names should not contain multiple periods because the
exported has converted the name in that case to use underscores.

Maybe it would be less confusing to not keep textures as used by
Blender in the PackageSources tree at all, and let the MSFS glTF
exporter copy it them the texture folder?
