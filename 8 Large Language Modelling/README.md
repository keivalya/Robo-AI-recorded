# **Large Language Modeling**

---

## **1. Introduction to Hugging Face**

Hugging Face is a platform offering state-of-the-art natural language processing (NLP) and machine learning (ML) tools, including:
- **Transformers**: Pretrained models for text, vision, and speech.
- **Datasets**: Curated datasets for machine learning.
- **Gradio**: Simplified UI for ML applications.

### **Key Components**
1. **Transformers**: Library for working with pretrained models like BERT, GPT, and T5.
2. **Datasets**: Access datasets for NLP, computer vision, and more.
3. **Gradio**: Build web-based GUIs for ML applications.

### **Install Hugging Face Libraries**
```bash
pip install transformers datasets gradio --quiet
```

---

## **2. Hands-on Project Development**

This section demonstrates developing an NLP model using Hugging Face for text classification.

### **Example: Sentiment Analysis**

```python
from transformers import pipeline

# Load a pretrained sentiment analysis pipeline
sentiment_pipeline = pipeline("sentiment-analysis")

# Test input
text = "I love using Hugging Face for NLP tasks!"

# Analyze sentiment
result = sentiment_pipeline(text)
print(result)
```

**Output Example:**
```plaintext
[{'label': 'POSITIVE', 'score': 0.9998}]
```

---

## **3. GUI for Chatbots**

We can create a chatbot using Hugging Face models and deploy it with **Gradio** for a user-friendly interface.

### **Code: Chatbot with Gradio**
```python
import gradio as gr
from transformers import AutoModelForCausalLM, AutoTokenizer

# Load a pretrained conversational model
model_name = "microsoft/DialoGPT-medium"
tokenizer = AutoTokenizer.from_pretrained(model_name)
model = AutoModelForCausalLM.from_pretrained(model_name)

# Chatbot function
def chat(input_text, chat_history=[]):
    new_user_input_ids = tokenizer.encode(input_text + tokenizer.eos_token, return_tensors="pt")
    bot_input_ids = torch.cat([torch.tensor(chat_history).long(), new_user_input_ids], dim=-1) if chat_history else new_user_input_ids

    chat_history = model.generate(bot_input_ids, max_length=1000, pad_token_id=tokenizer.eos_token_id)
    bot_response = tokenizer.decode(chat_history[:, bot_input_ids.shape[-1]:][0], skip_special_tokens=True)
    return bot_response, chat_history

# Build Gradio interface
with gr.Blocks() as chatbot_gui:
    chat_input = gr.Textbox(placeholder="Type a message...")
    chat_output = gr.Textbox()
    chat_history = gr.State([])
    chat_button = gr.Button("Send")

    def respond(user_input, chat_hist):
        bot_response, updated_history = chat(user_input, chat_hist)
        return bot_response, updated_history

    chat_button.click(respond, inputs=[chat_input, chat_history], outputs=[chat_output, chat_history])

# Launch chatbot GUI
chatbot_gui.launch()
```

**New Chatbot Interface from HF🤗** (source: https://www.gradio.app/docs/gradio/chatbot)
```python
import gradio as gr
import random
import time

with gr.Blocks() as demo:
    chatbot = gr.Chatbot(type="messages")
    msg = gr.Textbox()
    clear = gr.ClearButton([msg, chatbot])

    def respond(message, chat_history):
        bot_message = random.choice(["How are you?", "Today is a great day", "I'm very hungry"])
        chat_history.append({"role": "user", "content": message})
        chat_history.append({"role": "assistant", "content": bot_message})
        time.sleep(2)
        return "", chat_history

    msg.submit(respond, [msg, chatbot], [msg, chatbot])

if __name__ == "__main__":
    demo.launch()
```

---

## **4. Creating a Personal Assistant using Gradio**

We’ll use Hugging Face’s pretrained question-answering model to build a **personal assistant**.

### **Code: Personal Assistant**
```python
import gradio as gr
from transformers import pipeline

# Load a question-answering model
qa_pipeline = pipeline("question-answering", model="distilbert-base-uncased-distilled-squad")

# Question-answering function
def answer_question(context, question):
    result = qa_pipeline({"context": context, "question": question})
    return result["answer"]

# Gradio interface
with gr.Blocks() as assistant_gui:
    gr.Markdown("### Personal Assistant - Ask Me Anything")
    context = gr.Textbox(label="Context", placeholder="Provide a context (e.g., article, paragraph, etc.)")
    question = gr.Textbox(label="Question", placeholder="What do you want to know?")
    answer = gr.Textbox(label="Answer")
    ask_button = gr.Button("Ask")

    ask_button.click(answer_question, inputs=[context, question], outputs=answer)

# Launch the assistant GUI
assistant_gui.launch()
```

---

## **5. Creating a Text Summarization App (Similar to Quillbot)**

We’ll create a summarization tool using Hugging Face’s T5 model.

### **Code: Text Summarization App**
```python
import gradio as gr
from transformers import pipeline

# Load a summarization pipeline
summarization_pipeline = pipeline("summarization", model="t5-small")

# Summarization function
def summarize_text(input_text):
    summary = summarization_pipeline(input_text, max_length=100, min_length=30, do_sample=False)
    return summary[0]["summary_text"]

# Gradio interface
with gr.Blocks() as summarizer_gui:
    gr.Markdown("### Text Summarization App (Similar to Quillbot)")
    input_text = gr.Textbox(label="Input Text", placeholder="Paste the text you want to summarize here...")
    summary = gr.Textbox(label="Summary")
    summarize_button = gr.Button("Summarize")

    summarize_button.click(summarize_text, inputs=input_text, outputs=summary)

# Launch the summarizer GUI
summarizer_gui.launch()
```

---

## **Key Notes**

1. **Gradio**:
   - Simplifies the deployment of machine learning models with a GUI.
   - Supports interactive demos for chatbots, summarizers, and more.

2. **Pretrained Models**:
   - Use models from the Hugging Face Model Hub for specific tasks:
     - Sentiment Analysis: `"distilbert-base-uncased-finetuned-sst-2-english"`
     - Chatbot: `"microsoft/DialoGPT-medium"`
     - Summarization: `"t5-small"`

3. **Dependencies for Google Colab**:
   Ensure the following packages are installed:
   ```bash
   pip install transformers gradio torch --quiet
   ```

4. **GPU Runtime**:
   For faster inference, ensure GPU is enabled in Colab:
   - **Runtime > Change Runtime Type > Hardware Accelerator > GPU**.
