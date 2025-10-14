# TheraLink Firmware Guidelines

## Contexto do projeto
- Este repositório contém o firmware **TheraLink** para a placa BitDogLab (Raspberry Pi Pico W) utilizado no projeto final do programa Embarcatech.
- O objetivo do dispositivo é monitorar rapidamente o estado emocional e físico de grupos em ambientes offline, agregando leituras de batimentos (MAX30102), respostas subjetivas (energia, humor e ansiedade) e identificação por pulseira colorida (TCS34725) para guiar facilitadores.
- O firmware provê: criação de ponto de acesso Wi-Fi próprio, servidores DHCP/DNS/HTTP, espelhamento do display OLED SSD1306, APIs REST/CSV com estatísticas em tempo real, além do fluxo de coleta de dados com joystick, matriz de LEDs e sensores I²C.

## Diretrizes gerais
1. **Arquitetura e padrões**
   - Organize qualquer novo código seguindo princípios **SOLID** e favorecendo componentes coesos, com baixo acoplamento e injeção de dependências quando aplicável ao ambiente embarcado.
   - Prefira dividir responsabilidades em módulos bem nomeados dentro de `src/` e reutilizar abstrações existentes ao invés de duplicar lógica.
2. **Estilo de código**
   - Mantenha consistência com o estilo C utilizado no projeto: nomes de funções e variáveis em `snake_case`, comentários claros e foco em legibilidade.
   - Evite macros desnecessárias; prefira `static inline` para funções utilitárias pequenas.
3. **Documentação e comentários**
   - Sempre documente novas estruturas, funções públicas e fluxos complexos explicando a interação com sensores, rede ou UI.
   - Atualize `README.md` ou documentação relacionada quando adicionar funcionalidades relevantes para usuários ou facilitadores.
4. **Testes e validação**
   - Ao alterar comportamento crítico (rede, sensores, coleta de dados), inclua logs ou mecanismos de teste que permitam validar em hardware real ou por meio de mocks/simulações disponíveis.
5. **Pull requests**
   - Descreva claramente as mudanças, impactos e como validar. Informe dependências de hardware ou ajustes necessários no ambiente.

## Árvore de arquivos
- As diretivas deste arquivo aplicam-se a todo o repositório.
