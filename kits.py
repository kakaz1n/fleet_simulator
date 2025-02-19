class KitDePecas:
    def __init__(self, nome, pecas, armazem_origem):
        """
        Representa um kit de peças, que está armazenado em um local específico.
        :param nome: Nome do kit (ex: "Kit Motor", "Kit Rodas")
        :param pecas: Lista de peças pequenas dentro do kit
        :param armazem_origem: Nome do setor de Armazenamento onde está esse kit
        """
        self.nome = nome
        self.pecas = pecas
        self.armazem_origem = armazem_origem  # 🔹 Agora sabemos onde o kit está armazenado

    def __repr__(self):
        return f"KitDePecas(nome={self.nome}, pecas={self.pecas}, origem={self.armazem_origem})"

# # 🔹 Dicionário com Kits de Peças e seus Armazéns
# kits_disponiveis = {
#     "Kit Chassi": KitDePecas("Kit Chassi", ["estrutura metálica", "suportes", "solda"], "Armazenamento 1-1"),
#     "Kit Motor": KitDePecas("Kit Motor", ["parafuso", "biela", "bloco do motor"], "Armazenamento 1-2"),
#     "Kit Rodas": KitDePecas("Kit Rodas", ["pneu", "aro", "cubo de roda"], "Armazenamento 2-1"),
#     "Kit Cabine": KitDePecas("Kit Cabine", ["vidro", "assento", "painel"], "Armazenamento 2-2"),
#     "Kit Pintura": KitDePecas("Kit Pintura", ["tinta", "verniz"], "Armazenamento 3-1"),
# }
