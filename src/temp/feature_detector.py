import cv2
import os


test_path = os.path.realpath("src/test/scene_2.jpg")
template_path = os.path.realpath("src/templates/bottle.jpg")

img_ori = cv2.imread(test_path, 0)
template_ori = cv2.imread(template_path, 0)


def down_scale_image(image_input, times):
    layer = image_input.copy()
    gaussian_pyramid = [layer]
    for i in range(times + 1):
        layer = cv2.pyrDown(layer)
        gaussian_pyramid.append(layer)
    return gaussian_pyramid[times]


img = down_scale_image(img_ori, 2)
template = down_scale_image(template_ori, 2)

orb = cv2.ORB_create(nfeatures=1000)
keypoints1, description1 = orb.detectAndCompute(img, None)
keypoints2, description2 = orb.detectAndCompute(template, None)

img = cv2.drawKeypoints(img, keypoints1, None)
template = cv2.drawKeypoints(template, keypoints2, None)

# Brute-Force Matching
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(description1, description2)
matches = sorted(matches, key=lambda x: x.distance)

print(len(matches))
# for m in matches:
#     print(m.distance)

matching_results = cv2.drawMatches(img, keypoints1, template, keypoints2, matches, None)

cv2.imshow("img", img)
cv2.imshow("template", template)
cv2.imshow("Matching results", matching_results)
cv2.waitKey(0)
cv2.destroyAllWindows()
