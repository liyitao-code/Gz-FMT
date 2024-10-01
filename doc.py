#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from gensim.test.utils import common_texts
from gensim.models.doc2vec import Doc2Vec, TaggedDocument
from glob import glob
from scipy import spatial

DIR = "./models"




class SdfSimilarity:
    def __init__(self, directory=DIR):
        self.directory = directory
        self.content_list = []
        for file in glob(f"{self.directory}/*.sdf"):
            with open(file) as f:
                content = f.read()
                self.content_list.append(content.split())


        self.documents = [TaggedDocument(doc, [i]) for i, doc in enumerate(self.content_list)]
        self.model = Doc2Vec(self.documents)

        self.vectors = []
        for c in self.content_list:
            v = self.model.infer_vector(c)
            self.vectors.append(v)

        self.max_d = 0
        self.min_d = 100
        for i in self.vectors:
            for j in self.vectors:
                d = spatial.distance.cosine(i, j)
                if d != 0 and d < self.min_d:
                    self.min_d = d
                if d > self.max_d:
                    self.max_d = d

if __name__ == "__main__":
    sim = SdfSimilarity("./models")
