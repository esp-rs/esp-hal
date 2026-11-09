-- You must enable the exrc setting in neovim for this config file to be used.
local rust_analyzer = {
    cargo = {
        target = "{{ chip.rust_target }}",
        allTargets = false,
        --%if chip.xtensa
        extraEnv = { RUSTUP_TOOLCHAIN = "{{ rust_toolchain }}" },
        --%endif
    },
}

--%if option("neovim-rustaceanvim")
-- Note the rustaceanvim name of the language server is rust-analyzer with a dash.
vim.lsp.config("rust-analyzer", {
--%else
-- Note the neovim name of the language server is rust_analyzer with an underscore.
vim.lsp.config("rust_analyzer", {
--%endif
--%if chip.xtensa
	cmd = { "rustup", "run", "stable", "rust-analyzer" },
--%endif
    settings = {
        ["rust-analyzer"] = rust_analyzer,
    },
})
