#![allow(dead_code)]
use alloc::vec::Vec;
use defmt::Format;

#[derive(Copy, Clone, defmt::Format)]
pub enum DataTableError {
    RowIndexOutOfBounds,
    CannotGrowTable,
}

pub struct DataTable<'a, T, const M: usize> {
    header: [&'a str; M],
    data: Vec<T>,
}

impl<'a, T, const M: usize> DataTable<'a, T, M>
where
    T: Copy + Default + defmt::Format,
{
    pub fn new(header: [&'a str; M]) -> Self {
        Self {
            header,
            data: Vec::with_capacity(30),
        }
    }

    /// Get a reference to the row at the specified `index`. The `index`` must be less than the length of the table.
    pub fn get(&self, index: usize) -> Result<&T, DataTableError> {
        if index < self.length() {
            Result::Ok(&self.data[index])
        } else {
            Result::Err(DataTableError::RowIndexOutOfBounds)
        }
    }

    /// Append a row to the data table. The length of the table will be increased by one. The row
    /// will be copied into the data table. The row must implement the Copy trait. If the length
    /// of the table is equal to N, then the row will not be appended and an error will be returned.
    pub fn append(&mut self, row: T) -> &T {
        self.data.push(row);
        self.data.last().unwrap()
    }

    /// Erase the data table. The length of the table will be set to zero.
    pub fn erase(&mut self) {
        self.data.clear();
    }

    /// Get the length of the data table.
    pub fn length(&self) -> usize {
        self.data.len()
    }

    /// Get a reference to the headers of the data table.
    pub fn headers(&self) -> &[&'a str; M] {
        &self.header
    }

    /// Plots the data table. The data table will be plotted with rows on the horizontal axis and values
    /// on the vertical axis. The plot method will scan thrugh all the rows in th data table
    /// with the passed `value` function to determine the range of values to be plotted. The plot method will then
    /// scale the values to fit in the display area. The plot method will display the range of values
    /// on the vertical axis and the row index on the horizontal axis.
    ///
    /// # Arguments
    ///
    /// * `f` - The `defmt::Formatter` object that the graph should be printed to.
    /// * `value` - A function that gets called on each row in the data table to determine the value from that row to plot.
    /// This function must take a reference to the row type and return an `i32`. The mapping of the desired
    /// row value to the `i32` is for display purposes.
    pub fn plot(&self, f: &mut defmt::Formatter<'_>, value: fn(&T) -> i32) {
        // first we need to scan through the data to find the range of
        // values that we need to plot
        let mut min = i32::MAX;
        let mut max = i32::MIN;
        for row in self.data.iter() {
            let value = value(row);
            if value < min {
                min = value;
            }
            if value > max {
                max = value;
            }
        }
        let min_digits = Self::count_digits(min);
        let max_digits = Self::count_digits(max);
        let digits = if min_digits > max_digits {
            min_digits
        } else {
            max_digits
        };

        // now we can calculate the scale factor
        let scale = 1.0 / (max - min) as f32;
        const MAX_HEIGHT: i32 = 23;
        let display_height = if max - min > MAX_HEIGHT {
            MAX_HEIGHT
        } else {
            max - min
        };
        // now we can plot the data with rows on horizontal axis and values on vertical axis
        for h in (0..display_height + 1).rev() {
            if h == (display_height as f32 * (0 - min) as f32 / (max - min) as f32) as i32 {
                Self::write_n_spaces(digits - 1, f);
                defmt::write!(*f, "0 |");
            } else if h == display_height {
                Self::write_n_spaces(digits - max_digits, f);
                defmt::write!(*f, "{} |", max);
            } else if h == 0 {
                Self::write_n_spaces(digits - min_digits, f);
                defmt::write!(*f, "{} |", min);
            } else {
                Self::write_n_spaces(digits, f);
                defmt::write!(*f, " |");
            }
            for r in 0..self.length() {
                if let Result::Ok(row) = self.get(r) {
                    let value = value(row);
                    let scaled_value =
                        ((value - min) as f32 * scale * display_height as f32) as i32;
                    match scaled_value.cmp(&h) {
                        core::cmp::Ordering::Less => defmt::write!(*f, " "),
                        core::cmp::Ordering::Equal => defmt::write!(*f, "*"),
                        core::cmp::Ordering::Greater => defmt::write!(*f, "."),
                    }
                }
            }
            defmt::write!(*f, "\n");
        }
    }

    fn count_digits(value: i32) -> u32 {
        let mut n = value;
        let mut count = 0;
        if n < 0 {
            n = -n;
            count += 1; // for the '-' sign
        }
        loop {
            count += 1;
            n /= 10;
            if n == 0 {
                break;
            }
        }
        count
    }

    fn write_n_spaces(n: u32, f: &mut defmt::Formatter<'_>) {
        for _ in 0..n {
            defmt::write!(*f, " ");
        }
    }
}

impl<'a, T, const M: usize> Format for DataTable<'a, T, M>
where
    T: Copy + Default + Format,
{
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "DataTable {{\n");
        for h in 0..self.header.len() {
            defmt::write!(f, "{}", self.header[h]);
            if h < self.header.len() - 1 {
                defmt::write!(f, ",");
            }
        }
        defmt::write!(f, "\n");
        for row in self.data.iter() {
            defmt::write!(f, "{}\n", row);
        }
        defmt::write!(f, "}}");
    }
}

impl<'a, T, const M: usize> core::fmt::Display for DataTable<'a, T, M>
where
    T: Copy + Default + Format + core::fmt::Display,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "DataTable {{")?;
        for h in 0..self.header.len() {
            write!(f, "{}", self.header[h])?;
            if h < self.header.len() - 1 {
                write!(f, ",")?;
            }
        }
        writeln!(f)?;
        for row in self.data.iter() {
            writeln!(f, "{}", row)?;
        }
        write!(f, "}}")
    }
}
