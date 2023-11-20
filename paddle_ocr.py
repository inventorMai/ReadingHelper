import time
from PIL import Image

def ocr_PaddleOCR(img):
    from paddleocr import PaddleOCR, draw_ocr
    ocr = PaddleOCR(use_angle_cls=True, lang="ch")
    result = ocr.ocr(img, cls=True)
    for line in result:
        print(line)
    result = result[0]
    image = Image.open(img).convert('RGB')
    boxes = [line[0] for line in result]
    txts = [line[1][0] for line in result]
    scores = [line[1][1] for line in result]
    im_show = draw_ocr(image, boxes, txts, scores, font_path='simfang.ttf')
    im_show = Image.fromarray(im_show)
    im_show.save('result.jpg')

 
start_time = time.time()
ocr_PaddleOCR('test2.jpg')
end_time = time.time()
print('\n ==== OCR cost time: {} ===='.format(end_time-start_time))
