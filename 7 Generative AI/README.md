# Generative AI Concepts

---

## **1. Introduction to Generative AI**

Generative AI is a branch of artificial intelligence that focuses on creating models capable of generating new data that mimics a given distribution. Applications of Generative AI include text generation, image synthesis, style transfer, audio generation, and more.

### **Key Examples**
- Text: Large Language Models (GPT, T5)
- Images: Diffusion Models, GANs (e.g., Stable Diffusion, StyleGAN)
- Audio: Text-to-speech (e.g., WaveNet)

---

## **2. Encoder-Decoder Architecture**

The Encoder-Decoder architecture is primarily used in sequence-to-sequence (Seq2Seq) tasks like machine translation, summarization, and text generation.

### **Mathematics of Encoder-Decoder**
1. **Encoder**: Encodes input sequence $`x = (x_1, x_2, \dots, x_T)`$ into a context vector $`c`$.
   $`
   h_t = \text{Encoder}(x_t, h_{t-1}) \quad \text{for } t = 1, 2, \dots, T
   `$

   Final context vector:
   $`
   c = h_T
   `$

2. **Decoder**: Generates the output sequence $`y = (y_1, y_2, \dots, y_T)`$ based on $`c`$.
   $`
   s_t = \text{Decoder}(y_{t-1}, s_{t-1}, c)
   `$

   The output probability for each token:
   $`
   P(y_t | y_{<t}, c) = \text{softmax}(W \cdot s_t + b)
   `$

---

## **3. Large Language Models (LLMs)**

LLMs like GPT and BERT are built using **Transformer** architecture, which uses self-attention to capture relationships between tokens in a sequence.

### **Transformer Equation**
1. **Self-Attention Mechanism**:
   For a query $`Q`$, key $`K`$, and value $`V`$:
   $`
   \text{Attention}(Q, K, V) = \text{softmax}\left(\frac{QK^\top}{\sqrt{d_k}}\right)V
   `$
   where $`d_k`$ is the dimensionality of the keys.

2. **Feedforward Network**:
   After attention, each token passes through a feedforward neural network:
   $`
   \text{FFN}(x) = \text{ReLU}(W_1x + b_1)W_2 + b_2
   `$

3. **Transformer Blocks**:
   Each block has a multi-head attention mechanism:
   $`
   \text{Output} = \text{LayerNorm}(x + \text{Self-Attention}(x))
   `$

---

## **4. Diffusion Models**

Diffusion Models generate data by gradually transforming noise into meaningful data.

### **Mathematics of Diffusion Models**
1. **Forward Process**:
   A Gaussian noise is added to data over $`T`$ timesteps:
   $`
   q(x_t | x_{t-1}) = \mathcal{N}(x_t; \sqrt{\alpha_t}x_{t-1}, (1 - \alpha_t)\mathbf{I})
   `$
   where $`\alpha_t`$ is a variance schedule.

2. **Reverse Process**:
   The model learns to denoise the data:
   $`
   p_\theta(x_{t-1} | x_t) = \mathcal{N}(x_{t-1}; \mu_\theta(x_t, t), \Sigma_\theta(x_t, t))
   `$

3. **Loss Function**:
   The loss is based on the difference between the predicted and actual noise:
   $`
   \mathcal{L}_{\text{diffusion}} = \mathbb{E}_{x, \epsilon, t}\left[\|\epsilon - \epsilon_\theta(x_t, t)\|^2\right]
   `$

---

## **5. Hands-on How to Use State-of-the-Art LLMs Using Python APIs**

### **Using Hugging Face for Text Generation**
1. **Load Pretrained Model**:
   ```python
   from transformers import AutoTokenizer, AutoModelForCausalLM
   tokenizer = AutoTokenizer.from_pretrained("gpt2")
   model = AutoModelForCausalLM.from_pretrained("gpt2")
   ```

2. **Generate Text**:
   ```python
   input_text = "Once upon a time in a magical forest,"
   inputs = tokenizer(input_text, return_tensors="pt")
   outputs = model.generate(inputs["input_ids"], max_length=50, temperature=0.7)
   print(tokenizer.decode(outputs[0], skip_special_tokens=True))
   ```

---

## **6. Prompt Engineering**

Prompt Engineering is critical for optimizing the behavior of LLMs.

### **Examples of Prompts**
#### **Few-Shot Prompting**
Provide examples within the prompt:
```
Translate English to French:
1. Hello, how are you? -> Bonjour, comment ça va ?
2. I love programming. -> J'adore la programmation.
3. The weather is nice today. -> Le temps est agréable aujourd'hui.
```

#### **Chain-of-Thought Prompting**
Ask the model to reason step-by-step:
```
Q: If there are 5 apples and you take away 2, how many are left?

Step-by-step reasoning:
1. Start with 5 apples.
2. Take away 2 apples.
3. This leaves 3 apples.
```

---

## **7. Generative Adversarial Networks (GANs)**

GANs consist of two networks:
1. **Generator**: Generates fake data.
2. **Discriminator**: Distinguishes between real and fake data.

### **Mathematics of GANs**
1. **Objective Function**:
   GANs minimize the following:
   $`
   \min_G \max_D \mathbb{E}_{x \sim p_{\text{data}}}[\log D(x)] + \mathbb{E}_{z \sim p_z}[\log(1 - D(G(z)))]
   `$

2. **Generator Loss**:
   $`
   \mathcal{L}_G = -\mathbb{E}_{z \sim p_z}[\log D(G(z))]
   `$

3. **Discriminator Loss**:
   $`
   \mathcal{L}_D = -\mathbb{E}_{x \sim p_{\text{data}}}[\log D(x)] - \mathbb{E}_{z \sim p_z}[\log(1 - D(G(z)))]
   `$

---

## **8. Variational Autoencoders (VAEs)**

VAEs are probabilistic generative models that map data to a latent space distribution.

### **Mathematics of VAEs**
1. **Encoder**:
   Maps input $`x`$ to a latent space $`z`$ with parameters $`\mu`$ and $`\sigma`$:
   $`
   z \sim \mathcal{N}(\mu, \sigma^2)
   `$

2. **Decoder**:
   Reconstructs data $`\hat{x}`$ from $`z`$:
   $`
   \hat{x} \sim p_\theta(x | z)
   `$

3. **Loss Function**:
   The loss is the sum of reconstruction loss and KL divergence:
   $`
   \mathcal{L}_{\text{VAE}} = \mathbb{E}_{z \sim q_\phi(z|x)}[\log p_\theta(x|z)] - \text{KL}(q_\phi(z|x) \| p(z))
   `$