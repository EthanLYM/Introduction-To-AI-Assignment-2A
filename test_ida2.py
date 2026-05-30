from Search import cus2, parse_map, all_heuristics

origin, dests, nodes, adj, edges = parse_map('Map3.txt')

h_dict = all_heuristics(nodes, dests)
def h(n): return h_dict[n]

created_1 = [0]
def dfs_old(n, g, t, ps):
    f = g + h(n)
    if f > t: return f
    m = float('inf')
    for nb, ec in adj.get(n, []):
        if nb not in ps:
            created_1[0] += 1
            ps.add(nb)
            v = dfs_old(nb, g + ec, t, ps)
            ps.remove(nb)
            m = min(m, v)
    return m

created_2 = [0]
def dfs_new(n, g, t, ps, mg):
    f = g + h(n)
    if f > t: return f
    m = float('inf')
    for nb, ec in adj.get(n, []):
        if nb not in ps:
            if g + ec >= mg.get(nb, float('inf')):
                continue
            mg[nb] = g + ec
            created_2[0] += 1
            ps.add(nb)
            v = dfs_new(nb, g + ec, t, ps, mg)
            ps.remove(nb)
            m = min(m, v)
    return m

created_3 = [0]
def dfs_prune(n, g, t, ps):
    m = float('inf')
    for nb, ec in adj.get(n, []):
        if nb not in ps:
            f_nb = g + ec + h(nb)
            if f_nb > t:
                m = min(m, f_nb)
                continue
            created_3[0] += 1
            ps.add(nb)
            v = dfs_prune(nb, g + ec, t, ps)
            ps.remove(nb)
            m = min(m, v)
    return m

t1 = h(origin)
while t1 != float('inf'):
    t1 = dfs_old(origin, 0, t1, {origin})

t2 = h(origin)
while t2 != float('inf'):
    t2 = dfs_new(origin, 0, t2, {origin}, {origin: 0.0})

t3 = h(origin)
while t3 != float('inf'):
    t3 = dfs_prune(origin, 0, t3, {origin})

print("Old created:", created_1[0])
print("New created (min_g):", created_2[0])
print("Prune created:", created_3[0])
