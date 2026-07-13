pub trait EventWaiter {
    #[allow(async_fn_in_trait)]
    async fn wait(&self);
}
