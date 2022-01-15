#!/usr/bin/python3

import torch
import transformers
from transformers import GPTNeoForCausalLM, GPT2Tokenizer
import nltk
from nltk.tokenize.treebank import TreebankWordDetokenizer
from gingerit.gingerit import GingerIt
from random import randint

# For text cleanup and grammar check
d = TreebankWordDetokenizer()
parser = GingerIt()

# Load GPT Neo ML models
model = GPTNeoForCausalLM.from_pretrained("./gpt-neo-poetry")
tokenizer = GPT2Tokenizer.from_pretrained("./gpt-neo-poetry")

# ===========================================================================
# GENERATE PROPHECY
# ===========================================================================

def generate_prophecy(prompt):
    input_ids = tokenizer(prompt, return_tensors="pt").input_ids
    gen_tokens = model.generate(input_ids, do_sample=True, repetition_penalty=1.3, temperature=0.9, early_stopping=False, top_k=50, max_length=randint(60, 80))
    prophecy = tokenizer.batch_decode(gen_tokens)[0]
    prophecy = prophecy.replace(prompt, "")
    prophecy = clean_text(prophecy)
    return prophecy

def clean_text(text):
    # If first character is not a letter or number, it is removed
    while not text[0].isalnum():
        text = text[1:]
    sentences = nltk.sent_tokenize(text)
    sentences[0].strip()
    # If there's at least one complete sentence in generated prophecy
    if len(sentences) > 1:
        sentences.pop()
    sentences = d.detokenize(sentences)
    clean = parser.parse(sentences)['result']
    return clean

# ===========================================================================