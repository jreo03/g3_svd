# Godot 3 - Simulacra Vehicle Dynamics (pre-alpha)
<img width="1021" height="596" alt="image" src="https://github.com/user-attachments/assets/a30a9092-9a48-4f2e-a78b-3ad252b6f15b" />

vitavehicle alternative that's simpler to work with


currently in testing phase, coherent documentation will come soon

please replace/delete assets referred in borrowed_assets.txt when publishing your game using this project

feel free to submit any issues


## notes when porting godot 3 code to godot 4
- transform.xform(vector) -> transform*vector
- transform.xform_inv(vector) -> vector*transform
- global_translation -> global_transform.origin
