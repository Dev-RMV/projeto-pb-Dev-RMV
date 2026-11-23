# Evidências do TP4 — itens 3 e 4, pacote `tp4_extras` (geradas em 23/11/2026, WSL2 Ubuntu 22.04, Python 3.10)

Tudo aqui saiu de `./scripts/reproduzir_tp4_extras.sh` rodado numa pasta nova, só com o script, o `requirements_tp4_extras.txt` e o pacote `ros2_ws/src/tp4_extras`. Foram duas rodadas: a primeira com a placa de vídeo escondida (venv só de CPU, as quatro etapas, 680 s com a criação do venv) e a segunda só da etapa `cnn` com a RTX 4070 Ti SUPER visível (o script instalou o extra `and-cuda` do TensorFlow, 95 s). Versões: MetaDrive 0.4.3, TensorFlow 2.21.0 (Keras 3.12.4), OpenCV 5.1.0, numpy 2.2.6. A máquina também rodava as simulações do Gazebo nessa hora, então os tempos são um teto.

| Arquivo | O que mostra |
|---|---|
| `reproduzir_tp4_extras_saida.txt` | Saída das duas rodadas do script (sem as linhas de progresso do Keras) |
| `faixas_etapas.jpg` | **Item 3, faixas.** Três frames da câmera sintética do MetaDrive (640×360), cada um em quatro painéis: câmera, (a) HSV + máscara de cor (branco e amarelo, dentro do trapézio da pista), (b) Canny, (c) Hough com as faixas sobrepostas (vermelho: segmentos do `HoughLinesP`; ciano: linhas ajustadas; verde: as duas linhas e a área da faixa do veículo) |
| `faixas_reta.jpg`, `faixas_curva.jpg` | Resultado sobreposto numa reta e numa curva fechada (na curva a reta do Hough não acompanha a pista: é a limitação medida abaixo) |
| `faixas_resumo.json` | 300 frames no mapa `SCSCS`: as duas linhas da faixa do veículo em 86,7% (reta 94/101, curva 131/164, trecho inicial 35/35); erro contra a borda real da faixa no simulador, projetada na imagem a 8, 12 e 18 m: mediana 1,7 px à esquerda e 1,3 px à direita (p90 75 px e 15 px, por causa das curvas); 2,6 ms por frame |
| `formas_etapas.jpg` | **Item 3, formas.** Câmera, Hough Circle no frame inteiro (dispara também no asfalto e nas rodas), Harris no frame inteiro (cantos no carro e nas letras) e o resultado: região pela cor da placa, Hough Circle e Harris na silhueta, forma classificada |
| `formas_circular.jpg`, `formas_losango.jpg` | Resultado numa placa circular (círculo do Hough) e num losango (4 cantos do Harris) |
| `formas_resumo.json` | 250 frames no mapa `SCSS`, 14 placas na cena: 297 aparições com raio aparente ≥ 10 px, forma certa em 275 (92,6%): circular 76/87, losango 62/65, octogonal 61/65, triangular 76/80. Por tamanho: 10–15 px 97/119, acima de 15 px 178/178. 1 falso positivo em 250 frames |
| `dataset_amostras.jpg` | **Item 4, dataset.** Oito amostras de treino por classe (`via_livre`, `obstaculo`, `cruzamento`) |
| `dataset_resumo.json` | 500 frames 320×180, 167 / 167 / 166 por classe; treino 350 (sementes 1000–1025), validação 75 (3000–3008), teste 75 (5000–5006), mapas diferentes em cada partição; 159 obstáculos postos na faixa só para a foto (46 cones, 45 triângulos, 36 barreiras, 32 carros parados) e 8 do próprio trânsito; limiares dos rótulos; 389 s de coleta |
| `dataset_manifesto.csv` | Uma linha por frame: arquivo, partição, classe, semente, passo, origem e tipo do obstáculo, distância ao obstáculo e ao cruzamento, bloco e velocidade. As imagens (7,1 MB) não são versionadas: o script as gera de novo, e o manifesto sai idêntico (mesmo md5 em duas coletas) |
| `cnn_curvas.png` | **Item 4, CNN.** Accuracy e loss de treino e de validação sem e com augmentation, e as duas validações lado a lado (treino na GPU, 60 épocas) |
| `cnn_resumo.txt`, `cnn_resumo.json` | Treino na GPU (27 s + 30 s): métricas com os pesos da melhor época, diferenças entre as variantes e matrizes de confusão no teste |
| `cnn_resumo_cpu.txt` | O mesmo treino na CPU (114 s + 83 s), no venv sem as bibliotecas CUDA: o TensorFlow não acha GPU e o script segue na CPU |
| `cnn_historico.csv` | As quatro curvas, época a época, das duas variantes |
| `cnn_teste_perturbado.jpg` | Doze frames do teste perturbado (rotação de ±15°, zoom de 0,2 e flip sorteados uma vez) |

## O que a comparação com e sem augmentation mostrou

Mesma rede (4 blocos Conv + BatchNormalization, Dense 128, Dropout 0,5; 1,06 milhão de parâmetros), mesmos pesos iniciais, mesmo dataset.

| Rodada | Variante | Treino | Validação | Teste | Teste perturbado | Treino − teste | Loss no teste |
|---|---|---|---|---|---|---|---|
| GPU | sem augmentation | 1,000 | 0,973 | 0,827 | 0,640 | +0,173 | 0,564 |
| GPU | com augmentation | 0,897 | 0,973 | 0,880 | 0,827 | +0,017 | 1,018 |
| CPU | sem augmentation | 0,986 | 0,987 | 0,813 | 0,667 | +0,172 | 0,600 |
| CPU | com augmentation | 0,886 | 0,987 | 0,867 | 0,787 | +0,019 | 0,728 |

- **Menos memorização:** sem augmentation a rede chega a 99–100% no treino e fica 17 pontos acima do teste; com augmentation o treino fica em 89–90% e a diferença para o teste cai para 2 pontos.
- **Teste em pistas novas:** +5,3 pontos nas duas rodadas (62 → 66 e 61 → 65 acertos em 75). É pouco para 75 frames (cada frame vale 1,3 ponto): em outras três rodadas com os mesmos dados e hiperparâmetros a diferença foi +4,0, +2,7 e −2,7.
- **Teste perturbado:** é onde a augmentation rende de verdade, +18,7 e +12,0 pontos. A rede treinada sem augmentation cai para 64–67% quando a imagem vem girada ou com outro zoom.
- **O que piorou:** a loss no teste sobe com augmentation, mesmo com mais acertos, e a curva de validação oscila, caindo a ~0,5 em algumas épocas. Causa provável da oscilação (não isolada num experimento próprio): as médias móveis do BatchNormalization são estimadas nos lotes aumentados e a validação usa imagens limpas.
- A validação satura em 97–99% nas duas variantes; não separa uma da outra.
