import clip
import torch
from PIL import Image
import numpy as np


class CLIPMatcher:
    def __init__(self, model_name="ViT-B/16"):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        try:
            self.model, self.preprocess = clip.load(model_name, device=self.device)
            print(f"CLIP model {model_name} loaded on {self.device}")
        except Exception as e:
            print(f"Failed to load CLIP model: {str(e)}")
            print(
                f"Please make sure your model name is correct. Model list: {clip.available_models()}"
            )
            return

    def encode_text(self, text):
        """编码单个文本"""
        text_input = clip.tokenize([text]).to(self.device)
        with torch.no_grad():
            text_features = self.model.encode_text(text_input)
        return text_features.cpu().numpy()

    def encode_image(self, image_path):
        """编码单个图像"""
        try:
            image = Image.open(image_path).convert("RGB")
            image = self.preprocess(image).unsqueeze(0).to(self.device)
            with torch.no_grad():
                image_features = self.model.encode_image(image)
            return image_features.cpu().numpy()
        except Exception as e:
            print(f"图像编码失败: {image_path}, 错误: {str(e)}")
            return None

    def encode_images(self, image_paths):
        """批量编码多个图像（高效GPU批处理）"""
        images = []
        valid_paths = []

        # 加载并预处理图像
        for path in image_paths:
            try:
                image = Image.open(path).convert("RGB")
                image = self.preprocess(image)  # 不unsqueeze，后续统一堆叠
                images.append(image)
                valid_paths.append(path)
            except Exception as e:
                print(f"跳过无效图像 {path}, 错误: {str(e)}")

        if not images:
            return None, []

        # 合并为批次张量
        batch = torch.stack(images).to(self.device)

        # 提取特征
        with torch.no_grad():
            batch_features = self.model.encode_image(batch)

        return batch_features.cpu().numpy(), valid_paths

    def match_batch(self, text, image_paths):
        """批量计算文本与多个图像的相似度，返回排序结果"""
        # 编码文本
        text_features = self.encode_text(text)
        if text_features is None:
            return []

        # 批量编码图像
        batch_features, valid_paths = self.encode_images(image_paths)
        if batch_features is None:
            return []

        # 计算相似度矩阵 [num_images x 1]
        similarities = batch_features @ text_features.T

        # 将路径与相似度配对并排序
        sorted_results = sorted(
            zip(valid_paths, similarities[:, 0]), key=lambda x: x[1], reverse=True
        )

        return sorted_results


if __name__ == "__main__":
    matcher = CLIPMatcher()

    # 示例：批量匹配
    text_query = "iPhone"
    image_paths = [
        "/home/yutian/YanBot/last_seen_imgs/cellphone@1.png",
        "/home/yutian/YanBot/last_seen_imgs/cellphone@2.png",
        "/home/yutian/YanBot/last_seen_imgs/cellphone@3.png",
    ]

    # 执行批量匹配
    results = matcher.match_batch(text_query, image_paths)

    # 打印结果
    print(f"文本 '{text_query}' 与图像的相似度排序:")
    for rank, (path, score) in enumerate(results, 1):
        print(f"第{rank}名: {path} (相似度: {score:.4f})")
