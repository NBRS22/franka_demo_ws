import torch
from PIL import Image

from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor


assert torch.cuda.is_available()

print("GPU :", torch.cuda.get_device_name(0))
print("BF16 support :", torch.cuda.is_bf16_supported())


# Charger SAM3
print("Chargement modèle...")

model = build_sam3_image_model()
model = model.cuda()
model.eval()

processor = Sam3Processor(model)


image = Image.open("img.png").convert("RGB")


# Forcer toutes les opérations CUDA en BF16
with torch.autocast(
    device_type="cuda",
    dtype=torch.bfloat16
):
    with torch.no_grad():

        inference_state = processor.set_image(image)

        output = processor.set_text_prompt(
            state=inference_state,
            prompt="person"
        )


masks = output["masks"]
boxes = output["boxes"]
scores = output["scores"]

print("Nombre de masques :", len(masks))
print("Scores :", scores)
