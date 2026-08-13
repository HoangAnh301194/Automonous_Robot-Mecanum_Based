from typing import Optional, Sequence

import numpy as np
from rtmlib import RTMPose
from rtmlib.tools.pose_estimation.post_processings import convert_coco_to_openpose


import onnxruntime as ort


class BatchedRTMPose(RTMPose):
    def __init__(
        self,
        onnx_path: Optional[str] = None,
        onnx_model: Optional[str] = None,
        device: str = 'cpu',
        num_threads: int = 4,
        **kwargs,
    ):
        model_file = onnx_path or onnx_model
        kwargs.pop('backend', None)
        super().__init__(onnx_model=model_file, device=device, backend='onnxruntime', **kwargs)
        try:
            real_model_path = None
            if hasattr(self, 'onnx_model') and isinstance(self.onnx_model, str) and not self.onnx_model.startswith('http'):
                real_model_path = self.onnx_model
            elif hasattr(self, 'session') and hasattr(self.session, '_model_path'):
                real_model_path = self.session._model_path

            if real_model_path:
                opts = ort.SessionOptions()
                opts.intra_op_num_threads = num_threads
                opts.inter_op_num_threads = 2
                opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                providers = self.session.get_providers() if hasattr(self, 'session') else ['CPUExecutionProvider']
                self.session = ort.InferenceSession(real_model_path, sess_options=opts, providers=providers)
                print(f"[BatchedRTMPose] ONNX session successfully re-initialized with intra_op_num_threads={num_threads} on {real_model_path}")
            else:
                print(f"[BatchedRTMPose WARNING] Could not resolve local .onnx path. model_file={model_file}, onnx_model={getattr(self, 'onnx_model', None)}")
        except Exception as err:
            print(f"[BatchedRTMPose ERROR] Failed to enable ONNX multi-threading: {err}")

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
