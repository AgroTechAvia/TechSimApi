def ffs(x: int) -> int:
    """
    Find First Set bit.
    Возвращает индекс первого установленного бита (начиная с 1).
    Эквивалент функции ffs() из libc.
    """
    if x == 0:
        return 0
    return (x & -x).bit_length()