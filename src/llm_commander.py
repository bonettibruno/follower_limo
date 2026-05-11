#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
llm_commander.py
Nó ROS que traduz comandos em linguagem natural em sequências de ações JSON
usando uma LLM local via Hugging Face transformers.

Fluxo interno:
  texto NL → tokenizer.apply_chat_template() → tokens (IDs numéricos)
                                                    ↓
                                             modelo (transformer)
                                                    ↓
                                         tokens de saída (IDs)
                                                    ↓
                                        tokenizer.decode() → JSON string
                                                    ↓
                                            json.loads() → lista de ações
                                                    ↓
                                    publica em /task/actions (String)

Tópicos:
  Assina:  /task/command (String) — texto em linguagem natural
  Publica: /task/actions (String) — JSON array com ações para o task_executor
"""

import json
import re
import rospy
from std_msgs.msg import String

try:
    from transformers import AutoTokenizer, pipeline
    import torch
except ImportError:
    raise ImportError(
        "Dependencias nao instaladas. Execute:\n"
        "  pip install transformers torch accelerate")

# Prompt de sistema que instrui a LLM sobre as ações disponíveis.
# Mantido em inglês para melhor compatibilidade com modelos treinados em inglês.
SYSTEM_PROMPT = """\
You are a task planner for a mobile robot.
Given a command in natural language (Portuguese or English), return ONLY a valid JSON array with the action sequence.
Do NOT include any text before or after the JSON.

Available actions (with optional parameters):
- {"action": "go_forward", "distance_m": 1.0, "speed": 0.2}
- {"action": "rotate", "degrees": 90}
- {"action": "scan_area"}
- {"action": "search_object", "class_name": "sports ball", "timeout": 15}
- {"action": "approach_object", "class_name": "sports ball", "target_dist": 0.5}
- {"action": "stop"}
- {"action": "alert", "message": "text of the alert"}

Examples:
User: encontre a bola e me avise
Response: [{"action":"scan_area"},{"action":"search_object","class_name":"sports ball"},{"action":"approach_object","class_name":"sports ball","target_dist":0.5},{"action":"alert","message":"Bola encontrada!"}]

User: gire 90 graus e ande 1 metro
Response: [{"action":"rotate","degrees":90},{"action":"go_forward","distance_m":1.0}]

User: procure uma pessoa e se aproxime dela
Response: [{"action":"scan_area"},{"action":"search_object","class_name":"person"},{"action":"approach_object","class_name":"person","target_dist":1.0}]
"""


class LLMCommander:
    def __init__(self):
        rospy.init_node('llm_commander', anonymous=False)

        model_name      = rospy.get_param('~model_name',      'Qwen/Qwen2.5-0.5B-Instruct')
        max_new_tokens  = rospy.get_param('~max_new_tokens',  256)
        device_param    = rospy.get_param('~device',          'auto')

        rospy.loginfo("llm_commander: carregando '%s' ...", model_name)
        rospy.loginfo("  (primeira execucao baixa o modelo automaticamente)")

        # Escolhe device: 'auto' deixa o accelerate decidir (cuda se disponivel)
        if device_param == 'auto':
            torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32
        else:
            torch_dtype = torch.float16 if device_param == 'cuda' else torch.float32

        self.tokenizer = AutoTokenizer.from_pretrained(model_name)

        # pipeline abstrai o loop de inferência: tokenizar → gerar → decodificar
        self.pipe = pipeline(
            "text-generation",
            model=model_name,
            tokenizer=self.tokenizer,
            device_map="auto",
            torch_dtype=torch_dtype,
            max_new_tokens=max_new_tokens,
        )
        self.max_new_tokens = max_new_tokens

        rospy.loginfo("llm_commander: modelo pronto. Aguardando /task/command ...")

        self.pub_actions = rospy.Publisher('/task/actions', String, queue_size=1)
        rospy.Subscriber('/task/command', String, self._command_cb)

    # ── Callback de comando ──────────────────────────────────────────────────

    def _command_cb(self, msg):
        user_text = msg.data.strip()
        if not user_text:
            return

        rospy.loginfo("llm_commander: comando recebido: '%s'", user_text)

        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user",   "content": user_text},
        ]

        try:
            # apply_chat_template formata as mensagens no formato esperado pelo modelo
            # (ex: <|system|>...<|user|>...<|assistant|> para Qwen/Phi)
            prompt = self.tokenizer.apply_chat_template(
                messages, tokenize=False, add_generation_prompt=True)

            output = self.pipe(
                prompt,
                max_new_tokens=self.max_new_tokens,
                do_sample=False,          # determinístico para comandos robóticos
                pad_token_id=self.tokenizer.eos_token_id,
            )

            # generated_text contém prompt + resposta; remove o prompt
            generated = output[0]['generated_text']
            response  = generated[len(prompt):].strip()

            rospy.loginfo("llm_commander: resposta bruta:\n%s", response)

            actions = self._extract_json(response)
            if actions is None:
                rospy.logerr("llm_commander: nao foi possivel extrair JSON valido da resposta")
                return

            rospy.loginfo("llm_commander: publicando %d acoes em /task/actions", len(actions))
            self.pub_actions.publish(String(data=json.dumps(actions)))

        except Exception as e:
            rospy.logerr("llm_commander: erro durante inferencia: %s", e)

    # ── Extração de JSON ─────────────────────────────────────────────────────

    def _extract_json(self, text):
        """
        Extrai o primeiro JSON array encontrado na resposta da LLM.
        Necessário porque alguns modelos adicionam texto extra ao redor do JSON.
        """
        match = re.search(r'\[.*?\]', text, re.DOTALL)
        if not match:
            return None
        try:
            return json.loads(match.group())
        except ValueError:
            return None

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LLMCommander()
        node.run()
    except rospy.ROSInterruptException:
        pass
