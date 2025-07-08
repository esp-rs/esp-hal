use std::{collections::HashMap, error::Error, io};

use crossterm::{
    ExecutableCommand,
    event::{self, Event, KeyCode, KeyEventKind},
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use esp_config::{DisplayHint, Stability, Validator, Value};
use ratatui::{prelude::*, style::palette::tailwind, widgets::*};
use tui_textarea::{CursorMove, TextArea};

use crate::CrateConfig;

type AppResult<T> = Result<T, Box<dyn Error>>;

pub struct Repository {
    configs: Vec<crate::CrateConfig>,
    current_crate: Option<usize>,
}

enum Item {
    TopLevel(String),
    CrateLevel(crate::ConfigItem),
}

impl Item {
    fn title(&self, _width: u16, ui_elements: &UiElements) -> String {
        match self {
            Item::TopLevel(crate_name) => crate_name.clone(),
            Item::CrateLevel(config_option) => {
                let display_value = config_option
                    .option
                    .display_hint
                    .format_value(&config_option.actual_value);
                let default_indicator =
                    if config_option.actual_value == config_option.option.default_value {
                        ui_elements.default_value
                    } else {
                        ""
                    };

                let unstable_indicator = if config_option.option.stability == Stability::Unstable {
                    ui_elements.unstable
                } else {
                    ""
                };

                format!(
                    "{} ({}{}){}",
                    config_option.option.name, display_value, default_indicator, unstable_indicator
                )
            }
        }
    }

    fn help_text(&self) -> String {
        match self {
            Item::TopLevel(crate_name) => format!("The `{crate_name}` crate"),
            Item::CrateLevel(config_option) => config_option.option.description.clone(),
        }
        .replace("<p>", "")
        .replace("</p>", "\n")
        .replace("<br>", "\n")
        .to_string()
    }

    fn value(&self) -> Value {
        match self {
            Item::TopLevel(_) => unreachable!(),
            Item::CrateLevel(config_option) => config_option.actual_value.clone(),
        }
    }

    fn constraint(&self) -> Option<Validator> {
        match self {
            Item::TopLevel(_) => unreachable!(),
            Item::CrateLevel(config_option) => config_option.option.constraint.clone(),
        }
    }

    fn display_hint(&self) -> DisplayHint {
        match self {
            Item::TopLevel(_) => unreachable!(),
            Item::CrateLevel(config_option) => config_option.option.display_hint,
        }
    }
}

impl Repository {
    pub fn new(options: Vec<crate::CrateConfig>) -> Self {
        Self {
            configs: options,
            current_crate: None,
        }
    }

    fn current_level(&self) -> Vec<Item> {
        if self.current_crate.is_none() {
            Vec::from_iter(
                self.configs
                    .iter()
                    .map(|config| Item::TopLevel(config.name.clone())),
            )
        } else {
            Vec::from_iter(
                self.configs[self.current_crate.unwrap()]
                    .options
                    .iter()
                    .map(|option| Item::CrateLevel(option.clone())),
            )
        }
    }

    fn enter_group(&mut self, index: usize) {
        if self.current_crate.is_none() {
            self.current_crate = Some(index);
        }
    }

    fn up(&mut self) {
        if self.current_crate.is_some() {
            self.current_crate = None;
        }
    }

    fn set_current(&mut self, index: usize, new_value: Value) -> Result<(), String> {
        if self.current_crate.is_none() {
            return Ok(());
        }

        let crate_config = &mut self.configs[self.current_crate.unwrap()];
        let previous = crate_config.options[index].actual_value.clone();
        crate_config.options[index].actual_value = new_value;

        let res = validate_config(crate_config);
        if let Err(error) = res {
            crate_config.options[index].actual_value = previous;
            return Err(error.to_string());
        }

        Ok(())
    }

    // true if this is a configurable option
    fn is_option(&self, _index: usize) -> bool {
        self.current_crate.is_some()
    }

    // What to show in the list
    fn current_level_desc(&self, width: u16, ui_elements: &UiElements) -> Vec<String> {
        let level = self.current_level();

        level.iter().map(|v| v.title(width, ui_elements)).collect()
    }
}

pub fn init_terminal() -> AppResult<Terminal<impl Backend>> {
    enable_raw_mode()?;
    io::stdout().execute(EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(io::stdout());
    let terminal = Terminal::new(backend)?;
    Ok(terminal)
}

pub fn restore_terminal() -> AppResult<()> {
    disable_raw_mode()?;
    io::stdout().execute(LeaveAlternateScreen)?;
    Ok(())
}

struct UiElements {
    default_value: &'static str,
    unstable: &'static str,
    popup_highlight_symbol: &'static str,
}

struct Colors {
    header_bg: Color,
    normal_row_color: Color,
    help_row_color: Color,
    text_color: Color,

    selected_active_style: Style,
    edit_invalid_style: Style,
    edit_valid_style: Style,
    border_style: Style,
    border_error_style: Style,
}

impl Colors {
    const RGB: Self = Self {
        header_bg: tailwind::BLUE.c950,
        normal_row_color: tailwind::SLATE.c950,
        help_row_color: tailwind::SLATE.c800,
        text_color: tailwind::SLATE.c200,

        selected_active_style: Style::new()
            .add_modifier(Modifier::BOLD)
            .fg(tailwind::SLATE.c200)
            .bg(tailwind::BLUE.c950),
        edit_invalid_style: Style::new().add_modifier(Modifier::BOLD).fg(Color::Red),
        edit_valid_style: Style::new()
            .add_modifier(Modifier::BOLD)
            .fg(tailwind::SLATE.c200),
        border_style: Style::new()
            .add_modifier(Modifier::BOLD)
            .fg(Color::LightBlue),
        border_error_style: Style::new().add_modifier(Modifier::BOLD).fg(Color::Red),
    };
    const ANSI: Self = Self {
        header_bg: Color::DarkGray,
        normal_row_color: Color::Black,
        help_row_color: Color::DarkGray,
        text_color: Color::Gray,

        selected_active_style: Style::new()
            .add_modifier(Modifier::BOLD)
            .fg(Color::White)
            .bg(Color::Blue),
        edit_invalid_style: Style::new().add_modifier(Modifier::BOLD).fg(Color::Red),
        edit_valid_style: Style::new().add_modifier(Modifier::BOLD).fg(Color::Gray),
        border_style: Style::new()
            .add_modifier(Modifier::BOLD)
            .fg(Color::LightBlue),
        border_error_style: Style::new().add_modifier(Modifier::BOLD).fg(Color::Red),
    };
}

impl UiElements {
    const FANCY: Self = Self {
        default_value: " ⭐",
        unstable: " 🚧",
        popup_highlight_symbol: "▶️ ",
    };
    const FALLBACK: Self = Self {
        default_value: " *",
        unstable: " !",
        popup_highlight_symbol: "> ",
    };
}

pub struct App<'a> {
    repository: Repository,

    state: Vec<ListState>,

    confirm_quit: bool,

    editing: bool,
    textarea: TextArea<'a>,
    editing_constraints: Option<Validator>,
    input_valid: bool,

    showing_selection_popup: bool,
    list_popup: List<'a>,
    list_popup_state: ListState,

    show_error_message: bool,
    initial_message: Option<String>,

    ui_elements: UiElements,
    colors: Colors,
}

impl App<'_> {
    pub fn new(errors_to_show: Option<String>, repository: Repository) -> Self {
        let (ui_elements, colors) = match std::env::var("TERM_PROGRAM").as_deref() {
            Ok("vscode") => (UiElements::FALLBACK, Colors::RGB),
            Ok("Apple_Terminal") => (UiElements::FALLBACK, Colors::ANSI),
            _ => (UiElements::FANCY, Colors::RGB),
        };

        let mut initial_state = ListState::default();
        initial_state.select(Some(0));

        Self {
            repository,
            state: vec![initial_state],
            confirm_quit: false,
            editing: false,
            textarea: TextArea::default(),
            editing_constraints: None,
            input_valid: true,
            showing_selection_popup: false,
            list_popup: List::default(),
            list_popup_state: ListState::default(),
            show_error_message: errors_to_show.is_some(),
            initial_message: errors_to_show,
            ui_elements,
            colors,
        }
    }

    pub fn selected(&self) -> usize {
        if let Some(current) = self.state.last() {
            current.selected().unwrap_or_default()
        } else {
            0
        }
    }

    pub fn select_next(&mut self) {
        if let Some(current) = self.state.last_mut() {
            current.select_next();
        }
    }
    pub fn select_previous(&mut self) {
        if let Some(current) = self.state.last_mut() {
            current.select_previous();
        }
    }
    pub fn enter_menu(&mut self) {
        let mut new_state = ListState::default();
        new_state.select(Some(0));
        self.state.push(new_state);
    }
    pub fn exit_menu(&mut self) {
        if self.state.len() > 1 {
            self.state.pop();
        }
    }
}

impl App<'_> {
    pub fn run(
        &mut self,
        mut terminal: Terminal<impl Backend>,
    ) -> AppResult<Option<Vec<crate::CrateConfig>>> {
        loop {
            self.draw(&mut terminal)?;

            if let Event::Key(key) = event::read()? {
                if self.editing {
                    match key.code {
                        KeyCode::Enter if key.kind == KeyEventKind::Press => {
                            if !self.input_valid {
                                continue;
                            }

                            let selected = self.selected();
                            if self.repository.is_option(selected) {
                                let current = self.repository.current_level()[selected].value();
                                let text = self.textarea.lines().join("").to_string();
                                let mut value = current.clone();
                                if value.parse_in_place(&text).is_ok() {
                                    let set_res = self.repository.set_current(selected, value);
                                    self.handle_error(set_res);
                                } else {
                                    self.handle_error(Err("Invalid value".to_string()));
                                }
                            }

                            self.editing = false;
                        }
                        KeyCode::Esc => {
                            self.editing = false;
                        }
                        _ => {
                            if self.textarea.input(key) {
                                let selected = self.selected();
                                if self.repository.is_option(selected) {
                                    let current = self.repository.current_level()[selected].value();
                                    let text = self.textarea.lines().join("").to_string();
                                    let mut parsed_value = current.clone();
                                    let parse_res = parsed_value.parse_in_place(&text);
                                    let validator_failed = if let Some(constraint) =
                                        &self.editing_constraints
                                    {
                                        match parse_res {
                                            Ok(()) => constraint.validate(&parsed_value).is_err(),
                                            _ => false,
                                        }
                                    } else {
                                        false
                                    };

                                    let invalid = parse_res.is_err() || validator_failed;

                                    self.textarea.set_style(if invalid {
                                        self.colors.edit_invalid_style
                                    } else {
                                        self.colors.edit_valid_style
                                    });
                                    self.input_valid = !invalid;
                                }
                            }
                        }
                    }
                } else if self.showing_selection_popup && key.kind == KeyEventKind::Press {
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Esc => {
                            self.showing_selection_popup = false;
                        }
                        KeyCode::Down | KeyCode::Char('j') => self.list_popup_state.select_next(),
                        KeyCode::Up | KeyCode::Char('k') => self.list_popup_state.select_previous(),
                        KeyCode::Enter if key.kind == KeyEventKind::Press => {
                            let selected = self.selected();
                            if let Some(Validator::Enumeration(items)) = &self.repository.configs
                                [self.repository.current_crate.unwrap()]
                            .options[selected]
                                .option
                                .constraint
                            {
                                let set_res = self.repository.set_current(
                                    selected,
                                    Value::String(
                                        items[self.list_popup_state.selected().unwrap()].clone(),
                                    ),
                                );
                                self.handle_error(set_res);
                            }
                            self.showing_selection_popup = false;
                        }
                        _ => (),
                    }
                } else if self.show_error_message {
                    match key.code {
                        KeyCode::Enter if key.kind == KeyEventKind::Press => {
                            self.show_error_message = false;
                        }
                        _ => (),
                    }
                } else if key.kind == KeyEventKind::Press {
                    use KeyCode::*;

                    if self.confirm_quit {
                        match key.code {
                            Char('y') | Char('Y') => return Ok(None),
                            _ => self.confirm_quit = false,
                        }
                        continue;
                    }

                    match key.code {
                        Char('q') => self.confirm_quit = true,
                        Char('s') | Char('S') => return Ok(Some(self.repository.configs.clone())),
                        Esc => {
                            if self.state.len() == 1 {
                                self.confirm_quit = true;
                            } else {
                                self.repository.up();
                                self.exit_menu();
                            }
                        }
                        Char('h') | Left => {
                            self.repository.up();
                            self.exit_menu();
                        }
                        Char('l') | Char(' ') | Right | Enter => {
                            let selected = self.selected();
                            if self.repository.is_option(selected) {
                                let current = self.repository.current_level()[selected].value();
                                let constraint =
                                    self.repository.current_level()[selected].constraint();

                                match current {
                                    Value::Bool(value) => {
                                        let set_res = self
                                            .repository
                                            .set_current(selected, Value::Bool(!value));
                                        self.handle_error(set_res);
                                    }
                                    Value::Integer(_) => {
                                        let display_value = self.repository.current_level()
                                            [selected]
                                            .display_hint()
                                            .format_value(&current);
                                        self.textarea =
                                            make_text_area(&display_value, &self.colors);
                                        self.editing_constraints = constraint;
                                        self.editing = true;
                                    }
                                    Value::String(s) => match constraint {
                                        Some(Validator::Enumeration(items)) => {
                                            let selected_option =
                                                items.iter().position(|v| *v == s);
                                            self.list_popup =
                                                make_popup(items, &self.ui_elements, &self.colors);
                                            self.list_popup_state = ListState::default();
                                            self.list_popup_state.select(selected_option);
                                            self.showing_selection_popup = true;
                                        }
                                        _ => {
                                            self.textarea = make_text_area(&s, &self.colors);
                                            self.editing_constraints = None;
                                            self.editing = true;
                                        }
                                    },
                                }
                            } else {
                                self.repository.enter_group(self.selected());
                                self.enter_menu();
                            }
                        }
                        Char('j') | Down => {
                            self.select_next();
                        }
                        Char('k') | Up => {
                            self.select_previous();
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    fn draw(&mut self, terminal: &mut Terminal<impl Backend>) -> AppResult<()> {
        terminal.draw(|f| {
            f.render_widget(self, f.area());
        })?;

        Ok(())
    }

    fn handle_error(&mut self, result: Result<(), String>) {
        if let Err(error) = result {
            self.show_error_message = true;
            self.initial_message = Some(error);
        }
    }
}

fn make_text_area<'a>(s: &str, colors: &Colors) -> TextArea<'a> {
    let mut text_area = TextArea::new(vec![s.to_string()]);
    text_area.set_block(
        Block::default()
            .borders(Borders::ALL)
            .border_style(colors.border_style)
            .title("Input"),
    );
    text_area.set_style(colors.edit_valid_style);
    text_area.set_cursor_line_style(Style::default());
    text_area.move_cursor(CursorMove::End);
    text_area
}

fn make_popup<'a>(items: Vec<String>, ui_elements: &UiElements, colors: &Colors) -> List<'a> {
    List::new(items)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(colors.border_style)
                .title("Choose"),
        )
        .highlight_style(colors.selected_active_style)
        .highlight_symbol(ui_elements.popup_highlight_symbol)
        .repeat_highlight_symbol(true)
}

impl Widget for &mut App<'_> {
    fn render(self, area: Rect, buf: &mut Buffer) {
        let vertical = Layout::vertical([
            Constraint::Length(2),
            Constraint::Fill(1),
            Constraint::Length(self.help_lines(area)),
            Constraint::Length(self.footer_lines(area)),
        ]);
        let [header_area, rest_area, help_area, footer_area] = vertical.areas(area);

        self.render_title(header_area, buf);
        self.render_item(rest_area, buf);
        self.render_help(help_area, buf);
        self.render_footer(footer_area, buf);

        if self.editing {
            let area = Rect {
                x: 5,
                y: area.height / 2 - 2,
                width: area.width - 10,
                height: 3,
            };

            ratatui::widgets::Clear.render(area, buf);
            self.textarea.render(area, buf);
        }

        if self.showing_selection_popup {
            let area = Rect {
                x: 5,
                y: area.height / 2 - 3,
                width: area.width - 10,
                height: 6,
            };

            ratatui::widgets::Clear.render(area, buf);
            StatefulWidget::render(&self.list_popup, area, buf, &mut self.list_popup_state);
        }

        if self.show_error_message {
            let area = Rect {
                x: 5,
                y: area.height / 2 - 5,
                width: area.width - 10,
                height: 5,
            };

            let block = Paragraph::new(self.initial_message.as_ref().unwrap().clone())
                .style(self.colors.edit_invalid_style)
                .block(
                    Block::bordered()
                        .style(self.colors.border_error_style)
                        .padding(Padding::uniform(1)),
                );

            ratatui::widgets::Clear.render(area, buf);
            block.render(area, buf);
        }
    }
}

impl App<'_> {
    fn render_title(&self, area: Rect, buf: &mut Buffer) {
        Paragraph::new("esp-config")
            .bold()
            .centered()
            .render(area, buf);
    }

    fn render_item(&mut self, area: Rect, buf: &mut Buffer) {
        // We create two blocks, one is for the header (outer) and the other is for the
        // list (inner).
        let outer_block = Block::default()
            .borders(Borders::NONE)
            .fg(self.colors.text_color)
            .bg(self.colors.header_bg)
            .title_alignment(Alignment::Center);
        let inner_block = Block::default()
            .borders(Borders::NONE)
            .fg(self.colors.text_color)
            .bg(self.colors.normal_row_color);

        // We get the inner area from outer_block. We'll use this area later to render
        // the table.
        let outer_area = area;
        let inner_area = outer_block.inner(outer_area);

        // We can render the header in outer_area.
        outer_block.render(outer_area, buf);

        // Iterate through all elements in the `items` and stylize them.
        let items: Vec<ListItem> = self
            .repository
            .current_level_desc(area.width, &self.ui_elements)
            .into_iter()
            .map(|value| ListItem::new(value).style(Style::default()))
            .collect();

        // We can now render the item list
        // (look carefully, we are using StatefulWidget's render.)
        // ratatui::widgets::StatefulWidget::render as stateful_render
        let current_state = self
            .state
            .last_mut()
            .expect("State should always have at least one element");
        let list_widget = List::new(items)
            .block(inner_block)
            .highlight_style(self.colors.selected_active_style)
            .highlight_spacing(HighlightSpacing::Always);
        StatefulWidget::render(list_widget, inner_area, buf, current_state);
    }

    fn help_paragraph(&self) -> Option<Paragraph<'_>> {
        let selected = self
            .selected()
            .min(self.repository.current_level().len() - 1);
        let option = &self.repository.current_level()[selected];
        let help_text = option.help_text();
        if help_text.is_empty() {
            return None;
        }

        let help_block = Block::default()
            .borders(Borders::NONE)
            .fg(self.colors.text_color)
            .bg(self.colors.help_row_color);

        Some(
            Paragraph::new(help_text)
                .centered()
                .wrap(Wrap { trim: false })
                .block(help_block),
        )
    }

    fn help_lines(&self, area: Rect) -> u16 {
        if let Some(paragraph) = self.help_paragraph() {
            paragraph.line_count(area.width) as u16
        } else {
            0
        }
    }

    fn render_help(&self, area: Rect, buf: &mut Buffer) {
        if let Some(paragraph) = self.help_paragraph() {
            paragraph.render(area, buf);
        }
    }

    fn footer_paragraph(&self) -> Paragraph<'_> {
        let text = if self.confirm_quit {
            "Are you sure you want to quit? (y/N)"
        } else if self.editing {
            "ENTER to confirm, ESC to cancel"
        } else if self.showing_selection_popup {
            "Use ↓↑ to move, ENTER to confirm, ESC to cancel"
        } else if self.show_error_message {
            "ENTER to confirm"
        } else {
            "Use ↓↑ to move, ESC/← to go up, → to go deeper or change the value, s/S to save and generate, ESC/q to cancel"
        };

        Paragraph::new(text).centered().wrap(Wrap { trim: false })
    }

    fn footer_lines(&self, area: Rect) -> u16 {
        self.footer_paragraph().line_count(area.width) as u16
    }

    fn render_footer(&self, area: Rect, buf: &mut Buffer) {
        self.footer_paragraph().render(area, buf);
    }
}

pub(super) fn validate_config(config: &CrateConfig) -> Result<(), String> {
    let cfg: HashMap<String, Value> = config
        .options
        .iter()
        .map(|option| {
            (
                option.option.full_env_var(&config.name),
                option.actual_value.clone(),
            )
        })
        .collect();
    if let Err(error) = esp_config::do_checks(config.checks.as_ref(), &cfg) {
        return Err(error.to_string());
    }
    Ok(())
}

pub struct ConfigChooser {
    config_files: Vec<String>,
    state: ListState,
    colors: Colors,
}

impl ConfigChooser {
    pub fn new(config_files: Vec<String>) -> Self {
        let colors = match std::env::var("TERM_PROGRAM").as_deref() {
            Ok("vscode") => Colors::RGB,
            Ok("Apple_Terminal") => Colors::ANSI,
            _ => Colors::RGB,
        };

        let state = ListState::default().with_selected(Some(0));

        Self {
            config_files,
            state,
            colors,
        }
    }

    pub fn run(&mut self, mut terminal: Terminal<impl Backend>) -> AppResult<Option<String>> {
        loop {
            self.draw(&mut terminal)?;

            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press {
                    match key.code {
                        KeyCode::Char('q') => return Ok(None),
                        KeyCode::Esc => return Ok(None),
                        KeyCode::Char('j') | KeyCode::Down => {
                            self.state.select_next();
                        }
                        KeyCode::Char('k') | KeyCode::Up => {
                            self.state.select_previous();
                        }
                        KeyCode::Enter => {
                            let selected = self.state.selected().unwrap_or_default();
                            return Ok(Some(self.config_files[selected].clone()));
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    fn draw(&mut self, terminal: &mut Terminal<impl Backend>) -> AppResult<()> {
        terminal.draw(|f| {
            f.render_widget(self, f.area());
        })?;

        Ok(())
    }
}

impl Widget for &mut ConfigChooser {
    fn render(self, area: Rect, buf: &mut Buffer) {
        let vertical = Layout::vertical([
            Constraint::Length(2),
            Constraint::Fill(1),
            Constraint::Length(1),
        ]);
        let [header_area, rest_area, footer_area] = vertical.areas(area);

        Paragraph::new("esp-config")
            .bold()
            .centered()
            .render(header_area, buf);

        Paragraph::new("Choose a config to edit")
            .bold()
            .centered()
            .render(footer_area, buf);

        // We create two blocks, one is for the header (outer) and the other is for the
        // list (inner).
        let outer_block = Block::default()
            .borders(Borders::NONE)
            .fg(self.colors.text_color)
            .bg(self.colors.header_bg)
            .title_alignment(Alignment::Center);
        let inner_block = Block::default()
            .borders(Borders::NONE)
            .fg(self.colors.text_color)
            .bg(self.colors.normal_row_color);

        // We get the inner area from outer_block. We'll use this area later to render
        // the table.
        let outer_area = rest_area;
        let inner_area = outer_block.inner(outer_area);

        // We can render the header in outer_area.
        outer_block.render(outer_area, buf);

        // Iterate through all elements in the `items` and stylize them.
        let items: Vec<ListItem> = self
            .config_files
            .iter()
            .map(|value| ListItem::new(value.as_str()).style(Style::default()))
            .collect();

        // We can now render the item list
        // (look carefully, we are using StatefulWidget's render.)
        // ratatui::widgets::StatefulWidget::render as stateful_render
        let state = &mut self.state;
        let list_widget = List::new(items)
            .block(inner_block)
            .highlight_style(self.colors.selected_active_style)
            .highlight_spacing(HighlightSpacing::Always);
        StatefulWidget::render(list_widget, inner_area, buf, state);
    }
}
