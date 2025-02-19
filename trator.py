from kits import kits_disponiveis

class Trator:
    def __init__(self, celulas_disponiveis):
        """
        Inicializa um trator baseado nas células dinâmicas do `mission_manager.py`.
        """
        self.estrutura = {
            celula: kits_disponiveis[f"Kit {celula}"].pecas if f"Kit {celula}" in kits_disponiveis else []
            for celula in celulas_disponiveis if "Montagem" in celula or "Produção" in celula
        }
        self.status_celulas = {celula: "Aguardando Peças" for celula in self.estrutura}

    def receber_pecas(self, celula, pecas):
        """Recebe kits de peças e atualiza o status da célula de produção."""
        if celula in self.estrutura:
            faltam = set(self.estrutura[celula]) - set(pecas)
            if not faltam:
                self.status_celulas[celula] = "Completo"
            else:
                self.status_celulas[celula] = f"Faltam: {faltam}"

            print(f"🔧 Célula {celula} atualizada: {self.status_celulas[celula]}")

    def status(self):
        """Retorna o status da montagem do trator."""
        return self.status_celulas
