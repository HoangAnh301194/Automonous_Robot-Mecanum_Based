from typing import Optional, Sequence

import numpy as np
from rtmlib import RTMPose
from rtmlib.tools.pose_estimation.post_processings import convert_coco_to_openpose


class BatchedRTMPose(RTMPose):
    def __call__(
        self,
        image: np.ndarray,
        bboxes: Optional[Sequence[Sequence[float]]] = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        if bboxes is None or len(bboxes) == 0:
            bboxes = [[0, 0, image.shape[1], image.shape[0]]]

        inputs = []
        contexts = []
        for bbox in bboxes:
            pose_input, center, scale = self.preprocess(image, list(bbox))
            inputs.append(pose_input.transpose(2, 0, 1))
            contexts.append((center, scale))

        batch = np.ascontiguousarray(np.stack(inputs), dtype=np.float32)
        input_name = self.session.get_inputs()[0].name
        output_names = [output.name for output in self.session.get_outputs()]
        outputs = self.session.run(output_names, {input_name: batch})

        keypoints = []
        scores = []
        for index, (center, scale) in enumerate(contexts):
            person_outputs = [output[index:index + 1] for output in outputs]
            person_keypoints, person_scores = self.postprocess(
                person_outputs,
                center,
                scale,
            )
            keypoints.append(person_keypoints)
            scores.append(person_scores)

        keypoints_array = np.concatenate(keypoints, axis=0)
        scores_array = np.concatenate(scores, axis=0)
        if self.to_openpose:
            keypoints_array, scores_array = convert_coco_to_openpose(
                keypoints_array,
                scores_array,
            )
        return keypoints_array, scores_array
